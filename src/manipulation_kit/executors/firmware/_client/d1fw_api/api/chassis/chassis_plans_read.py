from http import HTTPStatus
from typing import Any

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_plans_read import ChassisPlansRead
from ...models.chassis_plans_read_response_200 import ChassisPlansReadResponse200
from ...models.error_envelope import ErrorEnvelope
from ...types import Response


def _get_kwargs(
    *,
    body: ChassisPlansRead,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "post",
        "url": "/v1/chassis/plans/read",
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisPlansReadResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisPlansReadResponse200.from_dict(response.json())

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
) -> Response[ChassisPlansReadResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisPlansRead,
) -> Response[ChassisPlansReadResponse200 | ErrorEnvelope]:
    """Read inactive normal-scene saved plans

     Raw task.json from a bounded named-scene export retains unknown fields. Writes require a fresh
    whole-collection revision and exact target content, refuse active or lossy collections, and validate
    fresh scene points. Timed drafts only; cyclic intervals, activation, execution, union and whole-file
    deletion are unsupported. One send, exact raw full-collection readback including positional
    renumbering; outcome_unknown must be reconciled without automatic retry or rollback. No atomic
    vendor compare-and-swap exists; external editors can race. Verified means stored content at read
    time, never physical success.

    Args:
        body (ChassisPlansRead): One explicitly named normal scene; union and paths are forbidden.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisPlansReadResponse200 | ErrorEnvelope]
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
    body: ChassisPlansRead,
) -> ChassisPlansReadResponse200 | ErrorEnvelope | None:
    """Read inactive normal-scene saved plans

     Raw task.json from a bounded named-scene export retains unknown fields. Writes require a fresh
    whole-collection revision and exact target content, refuse active or lossy collections, and validate
    fresh scene points. Timed drafts only; cyclic intervals, activation, execution, union and whole-file
    deletion are unsupported. One send, exact raw full-collection readback including positional
    renumbering; outcome_unknown must be reconciled without automatic retry or rollback. No atomic
    vendor compare-and-swap exists; external editors can race. Verified means stored content at read
    time, never physical success.

    Args:
        body (ChassisPlansRead): One explicitly named normal scene; union and paths are forbidden.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisPlansReadResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisPlansRead,
) -> Response[ChassisPlansReadResponse200 | ErrorEnvelope]:
    """Read inactive normal-scene saved plans

     Raw task.json from a bounded named-scene export retains unknown fields. Writes require a fresh
    whole-collection revision and exact target content, refuse active or lossy collections, and validate
    fresh scene points. Timed drafts only; cyclic intervals, activation, execution, union and whole-file
    deletion are unsupported. One send, exact raw full-collection readback including positional
    renumbering; outcome_unknown must be reconciled without automatic retry or rollback. No atomic
    vendor compare-and-swap exists; external editors can race. Verified means stored content at read
    time, never physical success.

    Args:
        body (ChassisPlansRead): One explicitly named normal scene; union and paths are forbidden.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisPlansReadResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    *,
    client: AuthenticatedClient | Client,
    body: ChassisPlansRead,
) -> ChassisPlansReadResponse200 | ErrorEnvelope | None:
    """Read inactive normal-scene saved plans

     Raw task.json from a bounded named-scene export retains unknown fields. Writes require a fresh
    whole-collection revision and exact target content, refuse active or lossy collections, and validate
    fresh scene points. Timed drafts only; cyclic intervals, activation, execution, union and whole-file
    deletion are unsupported. One send, exact raw full-collection readback including positional
    renumbering; outcome_unknown must be reconciled without automatic retry or rollback. No atomic
    vendor compare-and-swap exists; external editors can race. Verified means stored content at read
    time, never physical success.

    Args:
        body (ChassisPlansRead): One explicitly named normal scene; union and paths are forbidden.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisPlansReadResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            client=client,
            body=body,
        )
    ).parsed
