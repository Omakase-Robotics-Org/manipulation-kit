from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.error_envelope import ErrorEnvelope
from ...models.recorder_dumps_list_response_200 import RecorderDumpsListResponse200
from ...types import Response


def _get_kwargs() -> dict[str, Any]:

    _kwargs: dict[str, Any] = {
        "method": "get",
        "url": "/v1/recorder/dumps",
    }

    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | RecorderDumpsListResponse200 | None:
    if response.status_code == 200:
        response_200 = RecorderDumpsListResponse200.from_dict(response.json())

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
) -> Response[ErrorEnvelope | RecorderDumpsListResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ErrorEnvelope | RecorderDumpsListResponse200]:
    """List stored dumps, newest first

     Every stored dump's manifest, newest first. A dump whose manifest cannot be read is left out and
    logged rather than failing the listing. complete:false is an honest partial dump: the daemon was
    stopped between the snapshot written at the trigger and the state-log copy written after
    post_trigger_s.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | RecorderDumpsListResponse200]
    """

    kwargs = _get_kwargs()

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
) -> ErrorEnvelope | RecorderDumpsListResponse200 | None:
    """List stored dumps, newest first

     Every stored dump's manifest, newest first. A dump whose manifest cannot be read is left out and
    logged rather than failing the listing. complete:false is an honest partial dump: the daemon was
    stopped between the snapshot written at the trigger and the state-log copy written after
    post_trigger_s.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | RecorderDumpsListResponse200
    """

    return sync_detailed(
        client=client,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
) -> Response[ErrorEnvelope | RecorderDumpsListResponse200]:
    """List stored dumps, newest first

     Every stored dump's manifest, newest first. A dump whose manifest cannot be read is left out and
    logged rather than failing the listing. complete:false is an honest partial dump: the daemon was
    stopped between the snapshot written at the trigger and the state-log copy written after
    post_trigger_s.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | RecorderDumpsListResponse200]
    """

    kwargs = _get_kwargs()

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
) -> ErrorEnvelope | RecorderDumpsListResponse200 | None:
    """List stored dumps, newest first

     Every stored dump's manifest, newest first. A dump whose manifest cannot be read is left out and
    logged rather than failing the listing. complete:false is an honest partial dump: the daemon was
    stopped between the snapshot written at the trigger and the state-log copy written after
    post_trigger_s.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | RecorderDumpsListResponse200
    """

    return (
        await asyncio_detailed(
            client=client,
        )
    ).parsed
