from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.dump_create import DumpCreate
from ...models.error_envelope import ErrorEnvelope
from ...models.recorder_dumps_create_response_200 import RecorderDumpsCreateResponse200
from ...types import UNSET, Response, Unset


def _get_kwargs(
    *,
    body: DumpCreate | Unset = UNSET,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/recorder/dumps",
    }

    if not isinstance(body, Unset):
        _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ErrorEnvelope | RecorderDumpsCreateResponse200 | None:
    if response.status_code == 200:
        response_200 = RecorderDumpsCreateResponse200.from_dict(response.json())

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
) -> Response[ErrorEnvelope | RecorderDumpsCreateResponse200]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: DumpCreate | Unset = UNSET,
) -> Response[ErrorEnvelope | RecorderDumpsCreateResponse200]:
    """Take a dump now

     Runs the same two-phase pipeline an automatic trigger runs, with trigger.kind manual. post_trigger_s
    defaults to 0 because an operator is asking about what has already happened; a positive value blocks
    the request for that long. min_interval_s does not apply to a manual dump. A retention budget that
    is full and entirely pinned refuses with refused rather than evicting a pinned dump. The body is
    optional: every field of it has a default.

    Args:
        body (DumpCreate | Unset): `POST /v1/recorder/dumps`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | RecorderDumpsCreateResponse200]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    *,
    client: AuthenticatedClient | Client,
    body: DumpCreate | Unset = UNSET,
) -> ErrorEnvelope | RecorderDumpsCreateResponse200 | None:
    """Take a dump now

     Runs the same two-phase pipeline an automatic trigger runs, with trigger.kind manual. post_trigger_s
    defaults to 0 because an operator is asking about what has already happened; a positive value blocks
    the request for that long. min_interval_s does not apply to a manual dump. A retention budget that
    is full and entirely pinned refuses with refused rather than evicting a pinned dump. The body is
    optional: every field of it has a default.

    Args:
        body (DumpCreate | Unset): `POST /v1/recorder/dumps`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | RecorderDumpsCreateResponse200
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: DumpCreate | Unset = UNSET,
) -> Response[ErrorEnvelope | RecorderDumpsCreateResponse200]:
    """Take a dump now

     Runs the same two-phase pipeline an automatic trigger runs, with trigger.kind manual. post_trigger_s
    defaults to 0 because an operator is asking about what has already happened; a positive value blocks
    the request for that long. min_interval_s does not apply to a manual dump. A retention budget that
    is full and entirely pinned refuses with refused rather than evicting a pinned dump. The body is
    optional: every field of it has a default.

    Args:
        body (DumpCreate | Unset): `POST /v1/recorder/dumps`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ErrorEnvelope | RecorderDumpsCreateResponse200]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: DumpCreate | Unset = UNSET,
) -> ErrorEnvelope | RecorderDumpsCreateResponse200 | None:
    """Take a dump now

     Runs the same two-phase pipeline an automatic trigger runs, with trigger.kind manual. post_trigger_s
    defaults to 0 because an operator is asking about what has already happened; a positive value blocks
    the request for that long. min_interval_s does not apply to a manual dump. A retention budget that
    is full and entirely pinned refuses with refused rather than evicting a pinned dump. The body is
    optional: every field of it has a default.

    Args:
        body (DumpCreate | Unset): `POST /v1/recorder/dumps`.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ErrorEnvelope | RecorderDumpsCreateResponse200
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
