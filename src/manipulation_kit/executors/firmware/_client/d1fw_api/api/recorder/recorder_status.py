from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.error_envelope import ErrorEnvelope
from ...models.recorder_status_response_200 import RecorderStatusResponse200
from ...types import Response


def _get_kwargs() -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "get",
        "url": "/v1/recorder",
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | RecorderStatusResponse200 | None:
    if response.status_code == 200:
        response_200 = RecorderStatusResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ErrorEnvelope | RecorderStatusResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ErrorEnvelope | RecorderStatusResponse200]:
    """Drive-recorder status and what the ring currently holds

     What the recorder is configured to do, how much the in-memory state-log ring currently holds per
    namespace, how many dumps are stored against the retention budget, and the last trigger this daemon
    observed whether or not it became a dump. A daemon that runs no recorder answers unavailable rather
    than a status saying off: not taking dumps and having taken none are different facts.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | RecorderStatusResponse200]
    """

    kwargs = _get_kwargs()

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
) -> ErrorEnvelope | RecorderStatusResponse200 | None:
    """Drive-recorder status and what the ring currently holds

     What the recorder is configured to do, how much the in-memory state-log ring currently holds per
    namespace, how many dumps are stored against the retention budget, and the last trigger this daemon
    observed whether or not it became a dump. A daemon that runs no recorder answers unavailable rather
    than a status saying off: not taking dumps and having taken none are different facts.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | RecorderStatusResponse200
    """

    return sync_detailed(
        client=client,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ErrorEnvelope | RecorderStatusResponse200]:
    """Drive-recorder status and what the ring currently holds

     What the recorder is configured to do, how much the in-memory state-log ring currently holds per
    namespace, how many dumps are stored against the retention budget, and the last trigger this daemon
    observed whether or not it became a dump. A daemon that runs no recorder answers unavailable rather
    than a status saying off: not taking dumps and having taken none are different facts.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | RecorderStatusResponse200]
    """

    kwargs = _get_kwargs()

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
) -> ErrorEnvelope | RecorderStatusResponse200 | None:
    """Drive-recorder status and what the ring currently holds

     What the recorder is configured to do, how much the in-memory state-log ring currently holds per
    namespace, how many dumps are stored against the retention budget, and the last trigger this daemon
    observed whether or not it became a dump. A daemon that runs no recorder answers unavailable rather
    than a status saying off: not taking dumps and having taken none are different facts.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | RecorderStatusResponse200
    """

    return (
        await asyncio_detailed(
            client=client,
        )
    ).parsed
